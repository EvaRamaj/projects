"use strict";

import React from 'react';
import { Card, Button, FontIcon, TextField } from 'react-md';
import { withRouter } from 'react-router-dom'

import { AlertMessage } from './AlertMessage';
import Page from './Page';
import axios from 'axios';


class ItemForm extends React.Component {

    constructor(props) {
        super(props);
        if (this.props.item != undefined) {
            this.state = {
                name: props.item.names,
                condition: props.item.condition,
                description: props.item.description,
                categories: props.item.categories,
                price: props.item.price,
                availability: props.item.availability,
                owner_id: props.item.owner_id,
                photos: props.item.photos
            };
        } else {
            this.state = {
                name: '',
                condition: 'new',
                description: '',
                categories: props.data[0],
                price: '',
                availability: '',
                owner_id: '',
                photos: '',
                data: props.data,
                photoNames: [],
                selectedFile0: undefined,
            };
        }

        this.photoNames = [];
        this.handleChangeName = this.handleChangeName.bind(this);
        this.handleChangeCondition = this.handleChangeCondition.bind(this);
        this.handleChangeDescription = this.handleChangeDescription.bind(this);
        this.handleChangeCategories = this.handleChangeCategories.bind(this);
        this.handleChangePrice = this.handleChangePrice.bind(this);
        this.handleChangePhoto0 = this.handleChangePhoto0.bind(this);

        this.handleSubmit = this.handleSubmit.bind(this);
    }

    handleChangePhoto0(event) {
        this.setState(Object.assign({}, this.state, {selectedFile0: event.target.files[0]}));
        console.log(this.state);
    };

    handleChangeName(event) {
        this.setState(Object.assign({}, this.state, {name: event.target.value}));
    }

    handleChangeCondition(event) {
        console.log(event)
        this.setState(Object.assign({}, this.state, {condition: event.target.value}));
    }

    handleChangeDescription(event) {
        this.setState(Object.assign({}, this.state, {description: event.target.value}));
    }

    handleChangeCategories(event) {
        console.log(event.target.value);
        var value = this.state.data.filter(function (item) {
            if (item.name == event.target.value) {
                return item._id
            }
        });
        this.setState(Object.assign({}, this.state, {categories: value[0]._id}));
    }

    handleChangePrice(event) {
        this.setState(Object.assign({}, this.state, {price: event.target.value}));
    }


    handleSubmit(event) {
        event.preventDefault();

        let item = this.props.item;
        if (item == undefined) {
            item = {};
        }
        console.log("it", this.state);
        if (this.state.selectedFile0 != undefined) {
            const formData = new FormData();
            formData.append('photo', this.state.selectedFile0);
            axios.post('http://localhost:3000/photos', formData).then(
                response => {
                    this.photoNames.push(response.data.file.filename);
                    this.setState(Object.assign({}, this.state, {photoNames: this.photoNames}));
                    item.name = this.state.name;
                    item.condition = this.state.condition;
                    item.description = this.state.description;
                    item.categories = this.state.categories;
                    item.price = this.state.price;
                    item.photos = this.state.photoNames;
                    console.log('item', item);
                    this.props.onSubmit(item);
                }
            )
        }
        else {
            console.log(this.state.photoNames);
            item.name = this.state.name;
            item.condition = this.state.condition;
            item.description = this.state.description;
            item.categories = this.state.categories;
            item.price = this.state.price;
            item.photos = this.state.photoNames;

            this.props.onSubmit(item);
        }
    }

// Usage!
    render() {
        let categories = this.state.data;
        let optionItems = categories.map((category) =>
            <option key={category._id}>{category.name}</option>
        );

        return (
            <Page>
                <h3>Do you want to offer your asset?</h3>
                <h4>Upload your ID and soon your request will be evaluated! </h4>
                <form onSubmit={this.handleSubmit}>
                    <div className="form-row" onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()}>
                        <div className="form-group col-md-6">
                            <div className="container">
                                <label htmlFor="uname">Select a category*:</label>
                                <select className="custom-select custom-select-sm" required={true}
                                        onChange={this.handleChangeCategories}>
                                    {optionItems}
                                </select>
                            </div>
                            <label htmlFor="TitleField">Give a descriptive title*:</label>
                            <input
                                className="form-control"
                                placeholder="title"
                                id="TitleField"
                                type="text"
                                className="md-row"
                                required={true}
                                value={this.state.name}
                                onChange={this.handleChangeName}
                                errorText="Title is required"/>
                            <label htmlFor="file">Upload a photo*:</label>
                            <input type="file" required={false} onChange={this.handleChangePhoto0}/>


                        </div>
                        <div className="form-group col-md-6">
                            <div className="container">
                                <label htmlFor="psw">Condition*:</label>
                                <select className="custom-select custom-select-sm" required={true}
                                        onChange={this.handleChangeCondition}>
                                    <option value="new">New</option>
                                    <option value="good">Good</option>
                                    <option value="used">Used</option>
                                    <option value="bad">Bad</option>
                                </select>
                            </div>
                            <label htmlFor="DescriptionField">Details*:</label>
                            <input
                                className="form-control"
                                placeholder="description"
                                id="DescriptionField"
                                type="text"
                                className="md-row"
                                required={true}
                                value={this.state.description}
                                onChange={this.handleChangeDescription}/>


                            <label htmlFor="Price">Price*:</label>
                            <input
                                id="Price"
                                type="number"
                                className="md-row"
                                required={true}
                                value={this.state.price}
                                onChange={this.handleChangePrice}
                                errorText="Synopsis is required"/>

                        </div>
                    </div>
                    <div className="Upload_Button">
                        <div className="Align_Button">
                            <button className="Upload" id="submit" type="submit"
                                    raised primary className="md-cell md-cell--2">Upload
                            </button>
                            <AlertMessage
                                className="md-row md-full-width">{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
                        </div>
                    </div>

                </form>
            </Page>
        );
    }

}
export default withRouter(ItemForm);

// "use strict";
//
// import React from 'react';
// import { Card, Button, FontIcon, TextField } from 'react-md';
// import { withRouter } from 'react-router-dom'
//
// import { AlertMessage } from './AlertMessage';
// import Page from './Page';
// import axios from 'axios';
//
//
// const style = { maxWidth: 500 };
//
//
// class ItemForm extends React.Component {
//
//     constructor(props) {
//         super(props);
//         if(this.props.item != undefined) {
//             this.state = {
//                 name : props.item.names,
//                 condition : props.item.condition,
//                 description : props.item.description,
//                 categories: props.item.categories,
//                 price: props.item.price,
//                 availability: props.item.availability,
//                 owner_id: props.item.owner_id,
//                 photos: props.item.photos
//             };
//         } else {
//             this.state = {
//                 name : '',
//                 condition : 'new',
//                 description : '',
//                 categories: props.data[0],
//                 price: '',
//                 availability: '',
//                 owner_id: '',
//                 photos: '',
//                 data: props.data,
//                 photoNames: [],
//                 selectedFile0: undefined,
//             };
//         }
//
//         this.photoNames = [];
//         this.handleChangeName = this.handleChangeName.bind(this);
//         this.handleChangeCondition = this.handleChangeCondition.bind(this);
//         this.handleChangeDescription = this.handleChangeDescription.bind(this);
//         this.handleChangeCategories = this.handleChangeCategories.bind(this);
//         this.handleChangePrice = this.handleChangePrice.bind(this);
//         this.handleChangePhoto0 = this.handleChangePhoto0.bind(this);
//
//         this.handleSubmit = this.handleSubmit.bind(this);
//     }
//     handleChangePhoto0 (event){
//         this.setState(Object.assign({}, this.state, {selectedFile0: event.target.files[0]}));
//         console.log(this.state);
//     };
//
//     handleChangeName(value) {
//         this.setState(Object.assign({}, this.state, {name: value}));
//     }
//
//     handleChangeCondition(event) {
//         console.log(event)
//         this.setState(Object.assign({}, this.state, {condition: event.target.value}));
//     }
//
//     handleChangeDescription(value) {
//         this.setState(Object.assign({}, this.state, {description: value}));
//     }
//
//     handleChangeCategories(event) {
//         console.log(event.target.value);
//         var value = this.state.data.filter(function(item) {
//             if (item.name == event.target.value)
//             {
//                 return item._id
//             }
//         });
//         this.setState(Object.assign({}, this.state, {categories: value[0]._id}));
//     }
//
//     handleChangePrice(value) {
//         this.setState(Object.assign({}, this.state, {price: value}));
//     }
//
//
//     handleSubmit(event) {
//         event.preventDefault();
//
//         let item = this.props.item;
//         if(item == undefined) {
//             item = {};
//         }
//         if(this.state.selectedFile0 != undefined){
//             const formData = new FormData();
//             formData.append('photo', this.state.selectedFile0);
//             axios.post('http://localhost:3000/photos', formData).then(
//                 response => {
//                     this.photoNames.push(response.data.file.filename);
//                     this.setState(Object.assign({}, this.state, {photoNames: this.photoNames}));
//                     item.name = this.state.name;
//                     item.condition = this.state.condition;
//                     item.description = this.state.description;
//                     item.categories = this.state.categories;
//                     item.price = this.state.price;
//                     item.photos = this.state.photoNames;
//                     this.props.onSubmit(item);
//                 }
//             )
//         }
//         else {
//             console.log(this.state.photoNames);
//             item.name = this.state.name;
//             item.condition = this.state.condition;
//             item.description = this.state.description;
//             item.categories = this.state.categories;
//             item.price = this.state.price;
//             item.photos = this.state.photoNames;
//
//             this.props.onSubmit(item);
//         }
//     }
//
// // Usage!
//     render() {
//         let categories = this.state.data;
//         let optionItems = categories.map((category) =>
//             <option key={category._id}>{category.name}</option>
//         );
//
//         return (
//             <Page>
//                 <h2>Hey! Let's offer your assets!!</h2>
//                 <Card style={style} className="md-block-centered">
//                     <form className="md-grid" onSubmit={this.handleSubmit} onReset={() => this.props.history.goBack()}>
//                         <select required={true} onChange={this.handleChangeCategories}>
//                             {optionItems}
//                         </select>
//                         <TextField
//                             label="name"
//                             id="TitleField"
//                             type="text"
//                             className="md-row"
//                             required={true}
//                             value={this.state.name}
//                             onChange={this.handleChangeName}
//                             errorText="Title is required"/>
//                         <select required={true} onChange={this.handleChangeCondition}>
//                             <option value="new">New</option>
//                             <option value="good">Good</option>
//                             <option value="used">Used</option>
//                             <option value="bad">Bad</option>
//                         </select>
//                         <TextField
//                             label="description"
//                             id="RatingField"
//                             type="text"
//                             className="md-row"
//                             required={true}
//                             value={this.state.description}
//                             onChange={this.handleChangeDescription}/>
//                         <input type="file" required={false} onChange={this.handleChangePhoto0}/>
//                         <TextField
//                             label="price"
//                             id="SynopsisField"
//                             type="number"
//                             className="md-row"
//                             required={true}
//                             value={this.state.price}
//                             onChange={this.handleChangePrice}
//                             errorText="Synopsis is required"/>
//                         <Button id="submit" type="submit"
//                                 raised primary className="md-cell md-cell--2">Upload</Button>
//                         <AlertMessage className="md-row md-full-width" >{this.props.error ? `${this.props.error}` : ''}</AlertMessage>
//                     </form>
//                 </Card>
//             </Page>
//         );
//     }
// }
//
// export default withRouter(ItemForm);

