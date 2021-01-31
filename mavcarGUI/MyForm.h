#pragma once

namespace Auth {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for MyForm
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^ forwardb;
	private: System::Windows::Forms::Button^ leftb;
	protected:



	private: System::Windows::Forms::Button^ rightb;

	private: System::Windows::Forms::Button^ backwardb;
	private: System::Windows::Forms::Label^ label1;
	private: System::Windows::Forms::Label^ directionl;


	protected:


	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			System::ComponentModel::ComponentResourceManager^ resources = (gcnew System::ComponentModel::ComponentResourceManager(MyForm::typeid));
			this->forwardb = (gcnew System::Windows::Forms::Button());
			this->leftb = (gcnew System::Windows::Forms::Button());
			this->rightb = (gcnew System::Windows::Forms::Button());
			this->backwardb = (gcnew System::Windows::Forms::Button());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->directionl = (gcnew System::Windows::Forms::Label());
			this->SuspendLayout();
			// 
			// forwardb
			// 
			this->forwardb->BackgroundImage = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"forwardb.BackgroundImage")));
			this->forwardb->ForeColor = System::Drawing::Color::White;
			this->forwardb->Location = System::Drawing::Point(1092, 289);
			this->forwardb->Name = L"forwardb";
			this->forwardb->Size = System::Drawing::Size(83, 123);
			this->forwardb->TabIndex = 1;
			this->forwardb->Text = L"FORWARD";
			this->forwardb->UseVisualStyleBackColor = true;
			this->forwardb->Click += gcnew System::EventHandler(this, &MyForm::button1_Click);
			// 
			// leftb
			// 
			this->leftb->BackgroundImage = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"leftb.BackgroundImage")));
			this->leftb->ForeColor = System::Drawing::Color::White;
			this->leftb->Location = System::Drawing::Point(1003, 388);
			this->leftb->Name = L"leftb";
			this->leftb->Size = System::Drawing::Size(83, 88);
			this->leftb->TabIndex = 3;
			this->leftb->Text = L"LEFT";
			this->leftb->UseVisualStyleBackColor = true;
			this->leftb->Click += gcnew System::EventHandler(this, &MyForm::leftb_Click);
			// 
			// rightb
			// 
			this->rightb->BackgroundImage = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"rightb.BackgroundImage")));
			this->rightb->ForeColor = System::Drawing::Color::White;
			this->rightb->Location = System::Drawing::Point(1181, 388);
			this->rightb->Name = L"rightb";
			this->rightb->Size = System::Drawing::Size(83, 88);
			this->rightb->TabIndex = 4;
			this->rightb->Text = L"RIGHT";
			this->rightb->UseVisualStyleBackColor = true;
			this->rightb->Click += gcnew System::EventHandler(this, &MyForm::rightb_Click);
			// 
			// backwardb
			// 
			this->backwardb->BackgroundImage = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"backwardb.BackgroundImage")));
			this->backwardb->ForeColor = System::Drawing::Color::White;
			this->backwardb->Location = System::Drawing::Point(1092, 418);
			this->backwardb->Name = L"backwardb";
			this->backwardb->Size = System::Drawing::Size(83, 123);
			this->backwardb->TabIndex = 5;
			this->backwardb->Text = L"BACKWARD";
			this->backwardb->UseVisualStyleBackColor = true;
			this->backwardb->Click += gcnew System::EventHandler(this, &MyForm::backwardb_Click);
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->BackColor = System::Drawing::Color::Transparent;
			this->label1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 48, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->label1->ForeColor = System::Drawing::Color::Cornsilk;
			this->label1->Location = System::Drawing::Point(211, 366);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(540, 73);
			this->label1->TabIndex = 6;
			this->label1->Text = L"CAR IS MOVING:";
			this->label1->Click += gcnew System::EventHandler(this, &MyForm::label1_Click_1);
			// 
			// directionl
			// 
			this->directionl->AutoSize = true;
			this->directionl->BackColor = System::Drawing::Color::Transparent;
			this->directionl->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 24, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(0)));
			this->directionl->ForeColor = System::Drawing::Color::Cornsilk;
			this->directionl->Location = System::Drawing::Point(421, 439);
			this->directionl->Name = L"directionl";
			this->directionl->Size = System::Drawing::Size(0, 37);
			this->directionl->TabIndex = 7;
			this->directionl->TextAlign = System::Drawing::ContentAlignment::TopCenter;
			this->directionl->Click += gcnew System::EventHandler(this, &MyForm::directionl_Click);
			// 
			// MyForm
			// 
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::None;
			this->BackgroundImage = (cli::safe_cast<System::Drawing::Image^>(resources->GetObject(L"$this.BackgroundImage")));
			this->ClientSize = System::Drawing::Size(1277, 722);
			this->Controls->Add(this->directionl);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->backwardb);
			this->Controls->Add(this->rightb);
			this->Controls->Add(this->leftb);
			this->Controls->Add(this->forwardb);
			this->Name = L"MyForm";
			this->Text = L"MyForm";
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void button1_Click(System::Object^ sender, System::EventArgs^ e) {
		
		directionl->Text = "Forward";

	}
	private: System::Void leftb_Click(System::Object^ sender, System::EventArgs^ e) {

		directionl->Text = "Left";

	}



private: System::Void label1_Click_1(System::Object^ sender, System::EventArgs^ e) {
}
private: System::Void directionl_Click(System::Object^ sender, System::EventArgs^ e) {
}

private: System::Void backwardb_Click(System::Object^ sender, System::EventArgs^ e) {
	
	directionl->Text = "Backwards";

}
private: System::Void rightb_Click(System::Object^ sender, System::EventArgs^ e) {
	directionl->Text = "Right";
}
};
}
